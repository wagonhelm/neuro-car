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
typeDict = {
  'eyesClosed' : ['0', 'eyesClosed.csv'],
  'eyesOpen' : ['1', 'eyesOpen.csv']
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
  df = pd.read_csv(typeDict[key][1])
  
  # extraneous columns
  del df['Timestamp (ms)']
  del df['info']

  # iterate data points (rows)
  classInteger = typeDict[key][0]
  colNames = list(df)
  for i in range(df.shape[0]):
    line = classInteger + ' '
    for j in range(len(colNames)):
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