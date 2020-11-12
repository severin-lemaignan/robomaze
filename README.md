robomaze: help Wall-E to escape the maze
========================================

A game to learn programing.

**Check the `ros` branch for the ROS-focused version!**

Pre-requisites
--------------

- `sudo apt install python-flask python-jinja2`

Usage on local machine
----------------------

- start the Flask backend:

```sh
export FLASK_APP=backend/backend.py
export FLASK_DEBUG=1 # for debugging
flask run --host=0.0.0.0 --port=8080
```
- open a webbrowser and go to `http://localhost:8080`

Deployment on a production server
---------------------------------

- `sudo apt install nginx uwsgi uwsgi-plugin-python3`
- Install the configuration file for `nginx` provided in `etc/` in your
  `site-enabled` and restart the webserver (adjust the server name + www root as
  needed!)
- `uwsgi --ini robomaze.ini`
- open a webbrowser and go the website!


