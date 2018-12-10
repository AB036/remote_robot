from django.urls import path

from . import views

urlpatterns = [
    # path('<str:id>', views.StreamingVideoView.as_view(), name='streaming'),
    path('webcam', views.StreamingVideoView.as_view(), name='streaming_webcam'),
]
