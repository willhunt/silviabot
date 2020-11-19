import ROSLIB from 'roslib'

export const ros = new ROSLIB.Ros({
    url : webpackHotUpdate
            ? 'ws://localhost:9090'
            : 'ws://192.168.0.6:9090'
})
console.log(ros)

ros.on('connection', function() {
    console.log('Connected to websocket server.');
});
ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
});
ros.on('close', function() {
    console.log('Connection to websocket server closed.');
});
