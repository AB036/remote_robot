from django.shortcuts import render
from django.views import View
from django.http import JsonResponse


from django.http import HttpResponse
from django.template import loader

from control_board.socket_connection import SocketConnection


#socket_thread = SocketConnection()  # Creates the socket thread to connect in localhost with ROS


def index(request):
    #if not socket_thread.is_alive():
        #socket_thread.start()
    template = loader.get_template('control_board/index.html')
    return HttpResponse(template.render(request=request))


def move(request):
    direction = request.GET.get('direction', None)

    if direction == "up":
        socket_thread.send_command(0, 0)
    elif direction == "down":
        socket_thread.send_command(0, 1)
    elif direction == "left":
        socket_thread.send_command(0, 2)
    elif direction == "right":
        socket_thread.send_command(0, 3)

    return JsonResponse({'direction': direction})

