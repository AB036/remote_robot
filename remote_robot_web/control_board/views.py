from django.shortcuts import render
from django.views import View
from django.http import JsonResponse


from django.http import HttpResponse
from django.template import loader

from control_board.socket_connection import SocketConnection

def index(request):
    template = loader.get_template('control_board/index.html')
    return HttpResponse(template.render(request=request))


def move(request):
    direction = request.GET.get('direction', None)

    if direction == "up":
        SocketConnection.send_command(0, 0)
    elif direction == "down":
        SocketConnection.send_command(0, 1)
    elif direction == "left":
        SocketConnection.send_command(0, 2)
    elif direction == "right":
        SocketConnection.send_command(0, 3)

    return JsonResponse({'direction': direction})

