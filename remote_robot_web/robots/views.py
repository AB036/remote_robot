from django.views import generic
from django.http import StreamingHttpResponse

from .models import Robot


class StreamingVideoView(generic.DetailView):
    model = Robot
    template_name = 'robots/streaming.html'
