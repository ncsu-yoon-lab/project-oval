import csv

input_csv_file = 'gps_data.csv'
data_array = []
with open(input_csv_file, 'r') as csv_file:
    csv_reader = csv.reader(csv_file)
    for row in csv_reader:
        data_array.append(row)

def num_convert(num):
    print(num)
    if num[0] == '-':
        first_two = num[:3]
        rest = num[3:]

        rest = float(rest) / 60

        final_num = abs(float(first_two)) + rest

        return "-" + str(final_num)
    
    else:
        first_two = num[:2]
        rest = num[2:]

        rest = float(rest) / 60

        final_num = abs(float(first_two)) + rest

        return str(final_num)

data_array = (array for array in data_array if array) #this gets rid of all empty points

converted_data = []
for row in data_array:
    row[0] = num_convert(row[0])
    row[1] = num_convert(row[1])
    converted_data.append(row)
    

with open('gps_data_converted.csv', 'w', newline='') as file:
    csv_writer = csv.writer(file)
    csv_writer.writerows(converted_data)
    print('data written')


# data_array.append(new_row)
# with open(input_csv_file, 'w', newline='') as csv_file:
#     csv_writer = csv.writer(csv_file)
#     csv_writer.writerows(data_array)


