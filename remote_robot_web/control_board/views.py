from django.shortcuts import render

# Create your views here.

from django.http import HttpResponse
from django.template import loader
from django import forms


class ChatForm(forms.Form):
    message = forms.CharField(widget=forms.Textarea)


def index(request):
    form = ChatForm(request.POST or None)
    if form.is_valid():
        message = form.cleaned_data["message"]

    template = loader.get_template('control_board/index.html')
    return HttpResponse(template.render(request=request))


