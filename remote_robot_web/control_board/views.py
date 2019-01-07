from django.shortcuts import render
from django.views import View

# Create your views here.

from django.http import HttpResponse
from django.template import loader
from django import forms


class ChatForm(forms.Form):
    message = forms.CharField(widget=forms.Textarea)

def send_command(request) :
    command = request.GET['command']
    print("OOOOOOOOOKK"+command)
    template = loader.get_template('control_board/index.html')
    return HttpResponse(template.render(request=request))


def index(request):
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

