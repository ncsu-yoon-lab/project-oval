class GPS_Formatter():

    def coord_converter(num):
        if (num is not None and num != "None"):
            print(num)
            if(num == ''):
                return num
            elif num[0] == '-':
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


