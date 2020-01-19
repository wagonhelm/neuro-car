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

# file lists
eyesClosedToOpen = ['openClosed/closedopen.csv', 'openClosed/closedopen2.csv', 'openClosed/closedopen3.csv', 'openClosed/closedopen4.csv']
eyesClosed = ['nicClosed1.csv', 'nicClosed2.csv', 'nicClosed3.csv', 'nicClosed4.csv', 'micah_1_Closed.csv', 'micah_2_Closed1.csv', 'micah_2_Closed2.csv', 'JustinEyesClosed120sec01.csv', 'JustinEyesClosed120sec02.csv']

eyesOpenToClosed = ['openClosed/openclosed.csv', 'openClosed/openclosed2.csv', 'openClosed/openclosed3.csv', 'openClosed/openclosed4.csv']
eyesOpen = ['nicOpen1.csv', 'nicOpen2.csv', 'nicOpen3.csv', 'nicOpen4.csv', 'nicOpen5.csv', 'micah_1_Open.csv', 'micah_2_Open1.csv', 'micah_2_Open2.csv', 'JustinEyesOpen120sec01.csv', 'JustinEyesOpen120sec02.csv']

noFocus = ['JustinFocus001.csv', 'JustinFocus002.csv', 'JustinFocus003.csv', 'JustinFocus004.csv', 'JustinFocus005.csv', 'JustinFocus006.csv', 'JustinFocus007.csv', 'JustinFocus008.csv']
noFocus += ['nicNoFocus1.csv', 'nicNoFocus2.csv', 'nicNoFocus3.csv', 'nicNoFocus4.csv', 'nicNoFocus5.csv', 'nicNoFocus6.csv', 'TawsifNoFocus01.csv', 'TawsifNoFocus02.csv', 'TawsifNoFocus03.csv', 'TawsifNoFocus04.csv']
focus = ['JustinNoFocus001.csv', 'JustinNoFocus002.csv','JustinNoFocus003.csv','JustinNoFocus004.csv','JustinNoFocus005.csv','JustinNoFocus006.csv','JustinNoFocus007.csv','JustinNoFocus008.csv']
focus += ['TawsifFocus01.csv', 'TawsifFocus02.csv', 'TawsifFocus03.csv', 'TawsifFocus04.csv']

tired = ['nicTired1.csv', 'nicTired2.csv', 'nicTired3.csv', 'nicTired4.csv', 'nicTired5.csv', 'User03Level02.csv', 'User06Level02.csv']
mediumTired = ['User01Level01.csv', 'User02Level01.csv', 'User04Level01.csv']
rested = ['User01Level00.csv', 'ZubierLevel00.csv']

# list of class data files and associated numbers
dataRoot = './data/'
typeDict = {
  'eyesClosed' : ['0', eyesClosed],
  'eyesOpen' : ['1', eyesOpen],
}

"""
dataRoot = './data/roadAwareness/'
typeDict = {
  'noFocus' : ['0', noFocus],
  'focus' : ['1', focus]
}
"""

# output files
trainOut = open("train.libsvm", 'w')
evalOut = open("eval.libsvm", 'w')

# distribution metrics
totalTrain = 0
totalEval = 0

colCap = 120
gradLength = 1

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

    # aggregate consecutive rows into "gradient points"
    aggDf = pd.DataFrame(columns=range(0,colCap*gradLength))
    for r in range(0, partDf.shape[0], gradLength):
      if r + gradLength <= partDf.shape[0]:
        aggRow = []
        for s in range(0, gradLength):
          aggRow += list(partDf.iloc[r+s,:colCap])
        aggDf.loc[aggDf.shape[0]] = aggRow

    df = df.append(aggDf, ignore_index=True)
  print("{} instances of class {}".format(df.shape[0], typeDict[key][0]))

  # iterate data points (rows)
  classInteger = typeDict[key][0]
  colNames = list(df)
  for i in range(df.shape[0]):
    line = classInteger + ' '
    for j in range(colCap * gradLength):
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