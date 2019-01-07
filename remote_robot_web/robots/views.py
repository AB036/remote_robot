import cv2
from django.views import View
from django.http import StreamingHttpResponse, HttpResponseServerError, HttpResponse

from .models import Robot


class StreamingVideoView(View):
    class VideoCamera:
        def __init__(self):
            self.video = cv2.VideoCapture(0)

        def __del__(self):
            self.video.release()

        def get_frame(self):
            ret, image = self.video.read() #REPLACE BY THE MATRIX RECEIVING CODE
            ret, jpeg = cv2.imencode('.jpg', image)
            return jpeg.tobytes()

    # model = Robot
    # template_name = 'robots/streaming.html'

    def get(self, request):
        try:
            return StreamingHttpResponse(self.gen(self.VideoCamera()),
                                         content_type="multipart/x-mixed-replace;boundary=frame")
        except HttpResponseServerError as no_stream_exception:
            return HttpResponse("No stream available.")

    @staticmethod
    def gen(camera):
        while True:
            frame = camera.get_frame()
            yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')