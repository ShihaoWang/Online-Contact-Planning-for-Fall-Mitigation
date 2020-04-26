import os
import ipdb

PlanType = "RHP"
# PlanType = "OLP"
#
CWD = os.getcwd()
# FolderList = next(os.walk(CWD))[1]
# FolderList.sort()
ipdb.set_trace()
SuccessNo = 0
for i in range(1, 101):
    PlanResPath = CWD + '/' + str(i) + "/" + PlanType + "/PlanRes.txt"
    if os.path.isfile(PlanResPath):
        # Then we need to open To.txt file
         with open(PlanResPath, "r") as fp:
             Result = fp.readline()
             Result = Result.strip('\n')
             if Result =='1':
                 SuccessNo+=1
                 print "Success at: " + str(i) + " SuccessNo: " + str(SuccessNo)
    else:
        raise ValueError(PlanResPath)
print "I am Here!"
