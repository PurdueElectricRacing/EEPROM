import csv
csvfile = open('table.csv', 'w', newline='', encoding='utf-8')
csvwriter = csv.writer(csvfile)

with open("./putty.log",'r') as fid:
    for i in range(4000):
      byte_s = fid.read(1)
      if not byte_s:
         break

      int_value = ord(byte_s)
      row = [str(i), str(int_value), str(chr(int_value))]
      csvwriter.writerow(row)
      #binary_string = '{0:08b}'.format(int_value)
      print(row)

csvfile.close()

a = input("enter to quit")