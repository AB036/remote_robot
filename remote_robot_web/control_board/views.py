from django.shortcuts import render
from django.views import View
from django.http import HttpResponse, JsonResponse
from django import forms


# Create your views here.


class ChatForm(forms.Form):
    message = forms.CharField(widget=forms.Textarea)


class ControlBoardView(View):
    """View for index page."""
    template_name = 'control_board/index.html'
    form_class = ChatForm

    def get(self, request):
        return render(request, self.template_name)

    def post(self, request):
        form = self.form_class(request.POST or None)
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
            return HttpResponse(render(request, self.template_name))


def move(request):
    direction = request.GET.get('direction', None)

    if direction == "up":
        print('this is up')
    elif direction == "down":
        print('this is down')
    elif direction == "right":
        print('this is right')
    elif direction == "left":
        print('this is left')

    return JsonResponse({'direction': direction})
