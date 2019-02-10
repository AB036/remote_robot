import cv2
from django.views import View
from django.http import StreamingHttpResponse, HttpResponse

from control_board.socket_connection import SocketConnection


class StreamingVideoView(View):

    def get(self, request):
        """Generates HTTP response with image from video stream if available. Returns error otherwise."""
        try:
            generator = self.gen()
            if generator is not None:
                return StreamingHttpResponse(generator, content_type='multipart/x-mixed-replace;boundary=frame')
            else:
                return HttpResponse('No stream available.')
        except:
            return HttpResponse('No stream available.')

    @staticmethod
    def gen():
        """Get the current frame and returns it as generator."""
        frame = ''
        while frame is not None:
            image = SocketConnection.frame
            ret, jpeg = cv2.imencode('.jpg', image)
            frame = jpeg.tobytes()
            try:
                yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')
            except:
                return None
