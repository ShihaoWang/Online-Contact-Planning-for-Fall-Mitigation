import os
import ipdb
import sys

def main(argv):
    # Should be two numbers
    # First number is from * Second number is to
    CWD = os.getcwd()
    # ipdb.set_trace()
    cmd = "cd "
    cmd+= CWD + '/' + argv[0]
    cmd+=" && echo " + argv[1] + ">>To.txt"
    os.system(cmd)  # returns the exit code in unix

if __name__ == "__main__":
    main(sys.argv[1:])
