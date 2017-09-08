#!/usr/bin/env python 

#fp = open("dynamics_functionAnt.m")

import IPython
import re
from string import zfill, split
import sys

numbers = re.compile('\d+')
oper_string = "*+-/^(),"

import IPython

def find_exponent(s, loc):
    i = loc
    while i + 1 < len(s) and s[i + 1].isdigit():
        #while it's a digit
        i += 1
    return (i+1, int(s[loc + 1:i+1])) #this should be the integer
    
def find_matching_paren(s, loc):
    seen_count = 0
    while seen_count >=0:
        loc -= 1 #slide to the left
        elem = s[loc]
        if elem == ")":
            seen_count += 1
        elif elem == "(":
            seen_count -= 1
        
    return loc
    
def find_base(s, loc):
    if s[loc - 1] == ")": # if it's a paren
        idx = find_matching_paren(s, loc - 1)
    else: #consume non +/-/*///^ characters to the left
        idx = loc
        while idx -1 >= 0 and s[idx - 1] not in oper_string:
            idx -= 1
    return (idx, s[idx:loc])
        

def parse(s):
    #TODO: get count once, then for loop? Might be faster
    num = s.count("^")
    for i in range(num):
        loc = s.find("^")
        (end, exponent) = find_exponent(s, loc)
        (start, base) = find_base(s, loc)
        if end >= len(s):
            suffix = ''
        else:
            suffix = s[end:]
        prefix = s[:start]
        repl = ("pow(%s, %s)" %(base, exponent))
        s = "%s%s%s" % (prefix, repl, suffix)
        if (num - i) % 100 == 0:
            print num - i
    return s



#Second, replace the cosines and sines:
i = 0
#with open("dynamics_functionRobotArmWParams4Link.c") as infile, open("testfunction4Link.c", 'w') as outfile:

if __name__ == "__main__":
    with open(sys.argv[1]) as infile, open(sys.argv[2], 'w') as outfile:
        
        for line in infile:
            
            """
            count = line.count('^')
            while line.count > 0:
                #Find first '^'
                #TODO: assuming exactly one digit after exponent
                r_idx = line.find('^')
                prefix = line[:r_idx-1]
                suffix = line[r_idx + 1:]
                #Two options: either the character directly to the left is a ), or it's a variable.
                #case 1:
                if prefix[-1].isalnum():
                    for i in reversed(range(len(prefix))):
                        if not prefix[i].isalnum():
                            break # we found the end of the variable
                    if i != 0: #if we didn't go all the way to the beginning, we went one step too far:
                        i += 1
                    #now we found the beginning of the variable, so let's construct what the line should be:
                    #everything before that, pow(, the variable, ',', the exponent, ')', and then everything after the exponent
                    line = '%spow(%s,%d)%s'.format(line[:i-1], prefix[i:r_idx-1], int(prefix[r_idx + 1]), line[r_idx + 2:])
                
                #case 2:
            """
            
            
            """
            #i now represents the 
            #replace:
            internal = prefix[(l_idx + 1):(r_idx - 1)]
            before_internal
            line = '%s
            """
            line = parse(line)
            outfile.write(line)
            
        
