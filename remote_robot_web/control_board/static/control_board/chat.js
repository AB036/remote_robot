"use-strict";

// we instanciate a new web socket
var chatSocket = new WebSocket(
    'ws://' + window.location.host +
    '/ws/control_board/chat/');

// when a new message is sending, we add it to the chat
chatSocket.onmessage = function(e) {
    var data = JSON.parse(e.data);
    var message = data['message'];
    document.querySelector('#chat-log').value += (message + '\n');
};

// error message when the socket is unexpectedly closed
chatSocket.onclose = function(e) {
    console.log('Chat socket closed unexpectedly');
    // il faudrait peut-être également fermer la socket de la caméra
};


// send a message when we press enter
document.querySelector('#chat-message-input').focus();
document.querySelector('#chat-message-input').onkeyup = function(e) {
    if (e.keyCode === 13) {
        document.querySelector('#chat-message-submit').click();
    }
};

// send a message when we click on 'send'
document.querySelector('#chat-message-submit').onclick = function(e) {
    var messageInputDom = document.querySelector('#chat-message-input');
    var message = messageInputDom.value;
    // send via the socket as a json document
    chatSocket.send(JSON.stringify({
        'message': message
    }));

    messageInputDom.value = '';
};