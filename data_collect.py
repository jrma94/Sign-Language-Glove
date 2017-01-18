import serial, sys, getopt
import csv
import numpy

def main(argv):
	outputfile = ''
	try:
	  opts, args = getopt.getopt(argv,"ho:",["ofile="])
	except getopt.GetoptError:
	  print 'data_collect.py -o <outputfile>'
	  sys.exit(2)
	for opt, arg in opts:
	  if opt == '-h':
		 print 'data_collect.py-o <outputfile>'
		 sys.exit()
	  elif opt in ("-o", "--ofile"):
		 outputfile = arg

	ser = serial.Serial('COM6', 9600) 
	ser.flushInput()
	ser.flushOutput()

	result=numpy.array([], dtype = numpy.float32)

	counter = 0
	limit = 100

	lst=[]


	output_file = open("data\\" + outputfile + ".temoc", 'wb')
	output_file_csv = open("data\\" + outputfile + ".csv", 'w')

	while (counter < limit):
	  data_raw = ser.readline().strip()
	  if(counter == 0):
		data_raw = ser.readline().strip()
		
	  print(data_raw)
	  output_file_csv.write(data_raw + "\n")
	  data_delimited = data_raw.strip().split(",")
	  #print(data_delimited)
	  
	  temp = numpy.array(data_delimited, dtype='|S4')
	  result = temp.astype(numpy.float32)
	  result.tofile(output_file)
	  counter=counter + 1
	  


	output_file.close()
	output_file_csv.close()
	
if __name__ == "__main__":
   main(sys.argv[1:])