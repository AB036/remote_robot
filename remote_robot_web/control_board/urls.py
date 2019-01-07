from django.conf.urls import url


from . import views  # import views so we can use them in urls.


urlpatterns = [
    url(r'^ajax/move/$', views.move, name="ajax_moveUp")
]