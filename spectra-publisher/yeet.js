var app = require('express')();
var http = require('http').createServer(app);
var io = require('socket.io')(http);

const rosnodejs = require('rosnodejs');
const std_msgs = rosnodejs.require('std_msgs');

rosnodejs.initNode('/yeeter_node')
    .then(() => {
        const nh = rosnodejs.nh;
        const pub = nh.advertise('/muse_filtered_data', 'std_msgs/Float64MultiArray');

        // Set up an UBER-YEET subscriber to the MIGHTY YEET ROS MASTER'S classification of YEET:
        nh.subscribe('/svm/detection', 'std_msgs/Int32', (yeety_detection_data => {
            // We are receiving a new YEETY SVM detection! Yeet, am I right?!

            console.log("WE GOT A YEET IN THE HOUSE!", yeety_detection_data);

            io.emit('tasty_yeet', yeety_detection_data.data);

        }));

        app.get('/', function(req, res){
            res.sendFile(__dirname + '/client_index.html');
        });

        app.get('/bundle.js', function(req, res){
            res.sendFile(__dirname + '/dist/bundle.js');
        });

        io.on('connection', function(socket){
            console.log('a user connected');

            socket.on('yeet_data', function(data_that_has_been_yeeted){
                console.log("YEET!")

                let cleaner_yeet = []
                cleaner_yeet.push(...data_that_has_been_yeeted.psd[0])
                cleaner_yeet.push(...data_that_has_been_yeeted.psd[1]);
                cleaner_yeet.push(...data_that_has_been_yeeted.psd[2])
                cleaner_yeet.push(...data_that_has_been_yeeted.psd[3]);

                // console.log(cleaner_yeet)

                pub.publish(new std_msgs.msg.Float64MultiArray(cleaner_yeet));
            });
        });

        http.listen(3000, function(){
            console.log('listening on *:3000');
        });
    });
