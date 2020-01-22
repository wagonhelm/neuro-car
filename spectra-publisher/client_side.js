import {MUSE_SERVICE, MuseClient, zipSamples} from "muse-js";
import {
    bandpassFilter,
    epoch,
    fft,
    sliceFFT
} from "@neurosity/pipes";
import { catchError, multicast, take, takeUntil } from "rxjs/operators";
import {Subject, timer} from "rxjs";

const io = require('socket.io-client');

async function tryMuse(){
    document.getElementById('start_btn').disabled = true;

    const socket = await io.connect();

    const NUM_CHANS = 4;

    const museClient = new MuseClient();
    museClient.enableAux = false;

    console.log("muse connecting...");
    await museClient.connect();
    console.log("muse connected!");
    await museClient.start();
    console.log("started!");

    museClient.eegReadings.subscribe(reading => {
        // console.log("READ", reading);
    });

    const pipeSpectra = zipSamples(museClient.eegReadings).pipe(
        bandpassFilter({
            cutoffFrequencies: [2, 20],
            nbChannels: NUM_CHANS }),
        epoch({
            duration: 1024,
            interval: 40,
            samplingRate: 256
        }),
        fft({ bins: 256 }),
        sliceFFT([1, 30]),
        catchError(err => console.log(err))
    );

    const multicastSpectra = pipeSpectra.pipe(
        multicast(() => new Subject())
    );
    multicastSpectra.connect();

    const timer$ = timer(9999999999);

    // put selected observable object into local and start taking samples
    const localObservable = multicastSpectra.pipe(
        takeUntil(timer$)
    );

    localObservable.subscribe({
        next(x) {
            console.log("next", x);
            socket.emit('yeet_data', x)
        },
        error(err) { console.log(err); },
        complete() {
            console.log('Done publishing!');
        }
    });
}

window.tryMuse = tryMuse;
