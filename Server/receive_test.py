import requests

url = 'http://3.16.149.178/download/test.csv'

response = requests.get(url)

# Decode the byte string
csv_content = response.content.decode('utf-8')

# Split the content by lines
lines = csv_content.splitlines()

# Parse the lines into a 2D array
data = [line.split(',') for line in lines]

print(data)
