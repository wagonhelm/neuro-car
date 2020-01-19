#!/usr/bin/env python3

import pandas as pd
import random
import sys

# Arguments:
# evalPercentage - the percentage of data points to assign to the evaluation set [0,100]

# check terminal args
if len(sys.argv) != 2:
  print("Missing evaluation set percentage: eval in [0,100]")
  sys.exit(0)
assert(int(sys.argv[1]) in range(0, 100+1))

# percentage of data sent to evaluation set
evalPercent = int(sys.argv[1])
print("Percentage mapped to evaluation: {}".format(evalPercent))

# list of class data files and associated numbers
dataRoot = './data/'
typeDict = {
  'eyesClosed' : ['0', ['micah_1_Closed.csv']],
  'eyesOpen' : ['1', ['nicOpen1.csv', 'nicOpen2.csv', 'nicOpen3.csv', 'nicOpen4.csv', 'micah_1_Open.csv']]
}

# output files
trainOut = open("train.libsvm", 'w')
evalOut = open("eval.libsvm", 'w')

# distribution metrics
totalTrain = 0
totalEval = 0

# iterate classes
for key in typeDict:
  # load CSV
  df = pd.DataFrame()
  for name in typeDict[key][1]:
    partDf = pd.read_csv(dataRoot + name)

    if partDf.shape == (0,0):
      print("Invalid file: {}".format(dataRoot + name))
      sys.exit(0)

    # extraneous columns
    del partDf['Timestamp (ms)']
    del partDf['info']

    df = df.append(partDf, ignore_index=True) 

  # iterate data points (rows)
  classInteger = typeDict[key][0]
  colNames = list(df)
  for i in range(df.shape[0]):
    line = classInteger + ' '
    for j in range(120):
      line += str(j) + ':' + str(df[colNames[j]][i]) + ' '
    
    # split distribution
    if random.randint(0,100) < evalPercent:
      evalOut.write(line + '\n')
      totalEval += 1
    else:
      trainOut.write(line + '\n')
      totalTrain += 1

trainOut.close()
evalOut.close()

# print eval/train distribution
print("Distribution - total: {}, train: {}, eval: {}".format(totalEval + totalTrain, totalTrain, totalEval))
print("All done :)")