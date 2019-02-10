import cv2
from django.views import View
from django.http import StreamingHttpResponse, HttpResponse


class StreamingVideoView(View):
    class VideoCamera:
        def __init__(self):
            self.video = cv2.VideoCapture(0)

        def __del__(self):
            self.video.release()

        def get_frame(self):
            """Reads last frame of the video stream and return it compressed as jpeg."""
            ret, image = self.video.read()
            ret, jpeg = cv2.imencode('.jpg', image)
            return jpeg.tobytes()

    def get(self, request):
        """Generates HTTP response with image from video stream if available. Returns error otherwise."""
        try:
            generator = self.gen(self.VideoCamera())
            if generator is not None:
                return StreamingHttpResponse(generator, content_type='multipart/x-mixed-replace;boundary=frame')
            else:
                return HttpResponse('No stream available.')
        except:
            return HttpResponse('No stream available.')

    @staticmethod
    def gen(camera):
        """Get the current frame and returns it as generator."""
        frame = ''
        while frame is not None:
            frame = camera.get_frame()
            try:
                yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')
            except:
                return None
