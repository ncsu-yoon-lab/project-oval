from pydrive.auth import GoogleAuth
from pydrive.drive import GoogleDrive

gauth = GoogleAuth()
gauth.LocalWebserverAuth()
drive = GoogleDrive(gauth)

file_path = 'gps_data_converted.csv'

file_name = file_path

file = drive.CreateFile({'title': file_name})
file.SetContentFile(file_path)
file.Upload()

