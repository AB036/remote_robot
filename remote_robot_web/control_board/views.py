from django.shortcuts import render
from django.views import View
from django.http import JsonResponse


from django.http import HttpResponse
from django.template import loader


def index(request):
    template = loader.get_template('control_board/index.html')
    return HttpResponse(template.render(request=request))


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

