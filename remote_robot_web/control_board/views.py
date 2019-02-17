from django.shortcuts import render
from django.views import View
from django.http import JsonResponse, HttpResponseNotFound
from django import forms

from control_board.socket_connection import SocketConnection


# Create your views here.

class ChatForm(forms.Form):
    """Form for the chat, used in index page view."""
    message = forms.CharField(widget=forms.Textarea)


class ControlBoardView(View):
    """View for index page."""
    template_name = 'control_board/index.html'
    form_class = ChatForm

    def get(self, request):
        return render(request, self.template_name)


def move(request):
    """Gets the given direction and generates JSON response."""
    direction = request.GET.get('direction', None)

    if direction in ['up', 'down', 'right', 'left']:
        if direction == "up":
            SocketConnection.send_command(0, 0)
        elif direction == "down":
            SocketConnection.send_command(0, 1)
        elif direction == "left":
            SocketConnection.send_command(0, 2)
        elif direction == "right":
            SocketConnection.send_command(0, 3)

        return JsonResponse({'direction': direction})
    else:
        return HttpResponseNotFound('Wrong direction')
