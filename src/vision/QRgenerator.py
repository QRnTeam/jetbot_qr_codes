import pyqrcode

qr = pyqrcode.create('Stop')
qr.png('stop.png', scale=8)