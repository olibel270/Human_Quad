var express = require('express');
var app = express();
var server = require('http').createServer(app);
var io = require('socket.io')(server);
var zerorpc = require("zerorpc");

var client = new zerorpc.Client();
client.connect("tcp://127.0.0.1:4242");


// Playground on

client.invoke("get_battery_level", function(error, res, more) {
	console.log(res);
});
client.invoke("execute_trajectory", "ACTION, TRAJECTORY, TEST", function(error, res, more){
console.log(res);
});
client.invoke("get_position_link", "STATUS, POSITION, LINK", function(error, res, more){
console.log(res);
});


// Playground off
connections = [];

server.listen(8765, '192.168.0.114');
console.log('Server is running...');

io.sockets.on('connection', function (socket) {
    connections.push(socket);
    console.log('Connect: %s sockets are connected', connections.length)

    // Disconnect
    socket.on('disconnect', function (data) {
        connections.splice(connections.indexOf(socket), 1);
        console.log('Disconnect: %s sockets are connected', connections.length)
    });

    socket.on('NodeJS Server Port', function(data) {
        console.log(data);

        // Sending message back to client
        io.sockets.emit('iOS Client Port', {msg: 'You are now connected to the server!'});
    });

    socket.on('STATUS, BATTERY, LEVEL', function(data){
	//TODO: Set batteryRequested flag in another call
	batteryRequested = true;
	client.invoke("get_battery_level", function(error, res, more){
	while(batteryRequested == true){
		io.sockets.emit('iOS Client Port', {msg: res});
	}
	});
    });

   socket.on('STATUS, POSITION, LINK', function(data){
	   client.invoke("get_position_link", function(error, res, more){
		io.sockets.emit('iOS Client Port', {msg: res});
	   });
   });

    socket.on('ACTION, TRAJECTORY, APPROACH_AND_HOVER', function(data) {
        console.log(data);

	client.invoke("execute_trajectory", data, function(error, res, more) {});
        // Sending message back to client
        io.sockets.emit('iOS Client Port', {msg: 'SQUARE PATH CHOSEN'});
    });

    socket.on('ACTION, TRAJECTORY, CIRCLE', function(data) {
        console.log(data);

	client.invoke("execute_trajectory", data, function(error, res, more) {});
        // Sending message back to client
        io.sockets.emit('iOS Client Port', {msg: 'CIRCLE PATH CHOSEN'});
    });

    socket.on('ACTION, TRAJECTORY, HELIX', function(data) {
        console.log(data);

	client.invoke("execute_trajectory", data, function(error, res, more) {});
        // Sending message back to client
        io.sockets.emit('iOS Client Port', {msg: 'TRIANGLE PATH CHOSEN'});
    });

    socket.on('ACTION, TRAJECTORY, SQUARE_TRAJECTORY', function(data) {
        console.log(data);

	client.invoke("execute_trajectory", data, function(error, res, more) {});
        // Sending message back to client
        io.sockets.emit('iOS Client Port', {msg: 'PENTAGON PATH CHOSEN'});
    });
});
