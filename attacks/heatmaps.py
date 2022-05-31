import csv
import os
import re
import sys
import seaborn as sns

def countSysCalls(folder):
    sysCalls = {}
    regex = re.compile('.*csv$')
    for root, dirs, files in os.walk(folder):
        for file in files:
            if regex.match(file):
                with open(folder+"/"+file, "r") as systemCallsFile:
                    csvReader = csv.reader(systemCallsFile, delimiter=",")
                    next(csvReader)
                    for row in csvReader:
                        if row[2] not in sysCalls:
                            sysCalls[row[2]] = 1
                        else:
                            sysCalls[row[2]] += 1
                    return sysCalls


def main(argv):
    modes = ["normal", "repeat", "mimic", "confusion", "noise", "spoof", "freeze", "delay"]
    sysCallsList = []
    timestamp = argv[0]
    for mode in modes:
        sysCalls = countSysCalls("/data/"+timestamp+"/"+mode)
        sysCallsList.append(sysCalls)
    print(sysCallsList)
    

if __name__ == "__main__":
    main(sys.argv[1:])