import requests

url = 'http://3.16.149.178/upload'
files = {'file': open('C:\\Users\\malin\\Documents\\GitHub\\Server\\test.csv', 'rb')}
response = requests.post(url, files=files)
print(response.text)
