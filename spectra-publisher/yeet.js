import {MUSE_SERVICE, MuseClient, zipSamples} from "muse-js";
import {
    bandpassFilter,
    epoch,
    fft,
    sliceFFT
} from "@neurosity/pipes";
import {catchError, multicast, Subject, takeUntil, timer} from "rxjs";
import * as rosnodejs from "rosnodejs";

const noble = require('noble');
const bluetooth = require('bleat').webbluetooth;


async function tryMuse(){
    const NUM_CHANS = 4;

    const museClient = new MuseClient();
    museClient.enableAux = false;

    let device = await bluetooth.requestDevice({
        filters: [{ services: [MUSE_SERVICE] }]
    });
    const gatt = await device.gatt.connect();

    console.log("connecting...");
    await museClient.connect(gatt);
    console.log("connected!");
    await museClient.start();
    console.log("started!");

    const pipeSpectra = zipSamples(museClient.eegReadings).pipe(
        bandpassFilter({
            cutoffFrequencies: [2, 20],
            nbChannels: NUM_CHANS }),
        epoch({
            duration: 1024,
            interval: 100,
            samplingRate: 256
        }),
        fft({ bins: 256 }),
        sliceFFT([1, 30]),
        catchError(err => {
            console.log(err);
        })
    );

    const multicastSpectra = pipeSpectra.pipe(
        multicast(() => new Subject())
    );

    const timer$ = timer(10000);

    // put selected observable object into local and start taking samples
    const localObservable = multicastSpectra.pipe(
        takeUntil(timer$)
    );

    const nh = rosnodejs.nh;
    const pub = nh.advertise('/muse_filtered_data', 'std_msgs/Float64MultiArray');

    localObservable.subscribe({
        next(x) {
            pub.publish(Object.values(x));
            // logging is useful for debugging -yup
            console.log(x);
        },
        error(err) { console.log(err); },
        complete() {
            console.log('Done publishing!');
            process.exit()
        }
    });
}

noble.on('stateChange', (state) => {
    if (state === 'poweredOn') {
        connect();
    }
});
