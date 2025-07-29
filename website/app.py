from flask import Flask, render_template, request, jsonify
import sqlite3
import paho.mqtt.client as mqtt
import threading
import time
import json
from datetime import datetime
import signal
import sys
import os

app = Flask(__name__)
DATABASE = 'data.db'
