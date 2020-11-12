robomaze: help Wall-E to escape the maze
========================================

A game to learn programing.

**Check the `ros` branch for the ROS-focused version!**

Pre-requisites
--------------

- `sudo apt install python-flask python-jinja2`

Usage
-----

- start the Flask backend:

```sh
export FLASK_APP=backend/backend.py
export FLASK_DEBUG=1 # for debugging
flask run --host=0.0.0.0 --port=8080
```
- open a webbrowser and go to `http://localhost:8080`

Deployment on a production server
---------------------------------

- you need a webserver. A configuration file for `nginx` is provided in `etc/`

- start the webserver
- open a webbrowser and go the website


