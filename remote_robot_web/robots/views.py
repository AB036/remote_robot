import cv2
from django.views import View
from django.http import StreamingHttpResponse, HttpResponseServerError, HttpResponse

import numpy as np
from .models import Robot

from control_board.socket_connection import SocketConnection


def get_frame(self):
    ret, image = self.video.read() #R EPLACE BY THE MATRIX RECEIVING CODE
    ret, jpeg = cv2.imencode('.jpg', image)
    return jpeg.tobytes()


class StreamingVideoView(View):

    def get(self, request):
        try:
            return StreamingHttpResponse(self.gen(),content_type="multipart/x-mixed-replace;boundary=frame")
        except HttpResponseServerError as no_stream_exception:
            return HttpResponse("No stream available.")

    def gen(self):
        while True:
            imageRec = SocketConnection.frame
            ret, jpeg = cv2.imencode('.jpg', imageRec)
            frame = jpeg.tobytes()
            yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')
