"use strict";

function send_message() {

    console.log('vous avez envoyé le message : ');
    console.log($("#id_message").val());
    // insert message in chat
    $("#id_message").val("");

};

var url = "control_board/ajax/move/";

function moveUp() {

    $.ajax({
        url:url,
        type: 'get',
        data: {'direction': 'up'},
        success: function(data) {

            console.log('up');

        },
    });
};

function moveRight() {

    $.ajax({
        url:url,
        type: 'get',
        data: {'direction': 'right'},
        success: function(data) {

            console.log('right');

        },
    });
};

function moveLeft() {

    $.ajax({
        url:url,
        type: 'get',
        data: {'direction': 'left'},
        success: function(data) {

            console.log('left');

        },
    });
};

function moveDown() {

    $.ajax({
        url:url,
        type: 'get',
        data: {'direction': 'down'},
        success: function(data) {

            console.log('down');

        },
    });
};


function key_pressed(event) {

    if (event.keyCode == 38) {
        console.log("UP !");
    };

};