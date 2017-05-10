import sys
import glob
import os

if len(sys.argv) != 2:
    print 'Usage: %s <dirname>'
    sys.exit(1)

dirname = sys.argv[1]
#print dirname

cwd = os.getcwd()
os.chdir(dirname)
list_timestamp = []
for filename in glob.glob('*.png'):
    #print filename
    timestamp = filename.split('.png')[0]
    #print timestamp
    list_timestamp.append(timestamp)
os.chdir(cwd)

list_timestamp.sort(key=float)
#print list_timestamp
name_txt = '%s.txt' % dirname
print 'dumping: ' + name_txt
with open(name_txt, 'w') as f:
    for timestamp in list_timestamp:
        line = '%s %s/%s.png\n' % (timestamp, dirname, timestamp)
        #print line
        f.write(line)
