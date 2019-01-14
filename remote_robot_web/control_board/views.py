from django.shortcuts import render
from django.views import View
from django.http import JsonResponse

# Create your views here.

from django.http import HttpResponse
from django.template import loader
from django import forms

from control_board.Socket_connection import Socket_connection


class ChatForm(forms.Form):
    message = forms.CharField(widget=forms.Textarea)

socket_thread = Socket_connection()

def index(request):
    socket_thread.start()
    if request.method == 'POST':
        form = ChatForm(request.POST or None)
        if form.is_valid():
            message = form.cleaned_data["message"]
            html_msg = ""
            with open('control_board/templates/control_board/chat.html', 'r') as file_chat:
                for line in file_chat.readlines():
                    if line.strip() == "</div>":
                        break
                    html_msg += line

            html_msg += "<p>{}</p>".format(message) + "\n</div>\n</body>\n</html>"
            with open('control_board/templates/control_board/chat.html', 'w') as file_chat:
                file_chat.write(html_msg)

    template = loader.get_template('control_board/index.html')
    return HttpResponse(template.render(request=request))


def move(request):
    direction = request.GET.get('direction', None)

    if direction == "up":
        socket_thread.send_command(0, 0)
    elif direction == "down":
        socket_thread.send_command(0, 1)
    elif direction == "right":
        socket_thread.send_command(0, 2)
    elif direction == "left":
        socket_thread.send_command(0, 3)

    return JsonResponse({'direction': direction})

