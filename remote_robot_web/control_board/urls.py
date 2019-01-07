from django.urls import path


from . import views  # import views so we can use them in urls.


urlpatterns = [
    path('', views.IndexView.as_view(), name='index'),
    path('right-button', views.right_button, name='right_button'),
]