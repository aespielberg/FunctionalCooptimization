#!/usr/bin/env python 

#fp = open("dynamics_functionAnt.m")

import IPython
import re
from string import zfill, split
from ply import lex, yacc

numbers = re.compile('\d+')

from pyparsing import *
import IPython

"""
#ParserElement.enablePackrat()
# define arithmetic items
#identifier = Word(alphanums+'_')
def parse(s):
    identifier = Word(alphas, alphanums+'_') # not Regex(r'[a-zA-Z0-9_]+')
    real = Regex(r'\d+(\.\d*)?([Ee][+-]?\d+)?')
    oper = oneOf("* + - /")
    #oper = oneOf("* + - /")

    # define arithOperand as a Forward, since it could contain nested power expression
    arithOperand = Forward()
    arithExpr = arithOperand + ZeroOrMore(oper + arithOperand)
    groupedArithExpr = '(' + arithExpr + ')'

    # define expression for x^y, where x could be any kind of arithmetic term
    powerOp = Literal('^')
    powerExpr = (groupedArithExpr|real|identifier) + powerOp + real #order matters?
    powerExpr.setParseAction(lambda tokens: 'pow(%s,%s)' % (tokens[0], tokens[2]))

    # now define the possible expressions for arithOperand, including a powerExpr
    arithOperand <<= Optional('-') + (powerExpr | real | identifier | groupedArithExpr)

    # convert parsed list of strings to a single string
    groupedArithExpr.setParseAction(''.join)
    
    return arithExpr.transformString(s)
"""
oper_string = "*+-/^(),"

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

def replace_with_pow(matchgroup):
    s = matchgroup.group()
    #first, get the power:
    lastoccr_sre = list(numbers.finditer(s))[-1]
    lastoccr = lastoccr_sre.group()
    lastoccr_num = str(int(lastoccr))
    prefix = split(s, '^')[0]
    IPython.embed()
    return "pow(" + prefix + "," + lastoccr_num + ")"
    #then, get everything that preceeds ther '^'
    
    

def decrement(s):
    """ look for the first sequence of number(s) in a string and decrement """
    if numbers.findall(s):
        lastoccr_sre = list(numbers.finditer(s))[-1]
        lastoccr = lastoccr_sre.group()
        lastoccr_incr = str(int(lastoccr) + -1)
        return s[:lastoccr_sre.start()]+lastoccr_incr+s[lastoccr_sre.end():]

    return s
    
#Find string between two substrings
def find_between( s, first, last ):
    try:
        start = s.index( first ) + len( first )
        end = s.index( last, start )
        return s[start:end]
    except ValueError:
        return ""
      
#get the comma delimited sections  
def get_list_and_rebuild(s):
    between = find_between(line, '{', '}')
    elems = my_list = between.split(",")
    i = 0
    rebuild_list = 'double f[24] = {'
    for elem in elems:
        i+=1
        print i
        elem2 = parse(elem)
        #IPython.embed()
        rebuild_list += elem2 + ","
    return rebuild_list[:-1] + "};"

new_line = ""

replacements_outro = {}
for j in range(1, 25):
    new_line += "double s" + str(j) + " = sin(x" + str(j) + "*(1.0/2.0)); \n"
    replacements_outro["sin(x" + str(j) + "*(1.0/2.0))"] = "s" + str(j)
    new_line += "double c" + str(j) + " = cos(x" + str(j) + "*(1.0/2.0)); \n"
    replacements_outro["cos(x" + str(j) + "*(1.0/2.0))"] = "c" + str(j)
new_line += "double h1 = in1[0];\n"

replacements_outro['= ['] = '= {'
replacements_outro[']'] = '};'
replacements_outro[';'] = ','
replacements_outro['f ='] = 'double f[24] ='

replacements_final = {}
replacements_final[',,'] = ';'
replacements_final['4};'] = '4]' #TODO: Hack
replacements_final['};,'] = '};'

replacements = {}
replacements['./'] = '/'
replacements['.*'] = '*'
replacements['.^'] = '^' #TODO: Figure out what to replace this with for real
replacements['end'] = ''


replacements_intro = {'(' : '[', ',:)' : ']'}

#First, insert the other lines we needed





#Second, replace the cosines and sines:
i = 0
with open("dynamics_functionAnt.m") as infile, open("test2.c", 'w') as outfile:
    for line in infile:
        for src, target in replacements.iteritems():
            line = line.replace(src, target)
        if i < 76:
            for src, target in replacements_intro.iteritems():
                line = line.replace(src, target)
            line = decrement(line)
            line = "double " + line
            print line
            
        else:
            for src, target in replacements_outro.iteritems():
                line = line.replace(src, target) 
            #Now, first, get everything between the braces
            line = get_list_and_rebuild(line)
            
            
            #line = re.sub(r'[a-zA-Z_]+[0-9]+\^[0-9]+', replace_with_pow, line)
            #line = re.sub(r'\(([a-zA-Z_]+[0-9]+\+?-?\*?)+\)\^[0-9]+', replace_with_pow, line)
            #line = re.sub(r'\(.+?\)\^[0-9]+', replace_with_pow, line)
            #line = re.sub(r'\((.+?\(.+?\).+?\)+)\^[0-9]+', replace_with_pow, line)                 
        for src, target in replacements_final.iteritems(): #TODO: postprocessing hack
            line = line.replace(src, target)
        i = i + 1
        outfile.write(line)
        
#Finally, add the proper header and footer:
i = 0
with open("test2.c") as infile, open("test3.c", 'w') as outfile:
    #First write the header
    outfile.write('#include "mex.h"\n')
    outfile.write('#include <math.h>\n')
    outfile.write('void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]){\n')
    outfile.write('double * in1 = mxGetPr(prhs[0]);\n')
    outfile.write('double * in2 = mxGetPr(prhs[1]);\n')
    outfile.write('double * in3 = mxGetPr(prhs[2]);\n')
    outfile.write('double * in4 = mxGetPr(prhs[3]);\n')
    outfile.write('double * in5 = mxGetPr(prhs[4]);\n')
    outfile.write('double * in6 = mxGetPr(prhs[5]);\n')
    

    
    for line in infile:
        if i > 6:
            outfile.write(line)
        i = i + 1
   
    #Then write the footer TODO: set the output variable
    outfile.write("plhs[0] = mxCreateDoubleMatrix(24, 1, mxREAL);\n")
    outfile.write("double * outMatrix = mxGetPr(plhs[0]);\n")
    #outfile.write("double * data = (double *) mxGetData(plhs[0]);\n")
    outfile.write("int i; \n")
    outfile.write("for (i = 0; i < 24; i++) { \n outMatrix[i] = f[i]; \n }\n")
    
    
    outfile.write("}")
    
f = open("test3.c", "r")
contents = f.readlines()
f.close()

contents.insert(78, new_line)



f = open("test4.c", "w")
contents = "".join(contents)
f.write(contents)
f.close()



"""
for i, line in enumerate(fp):
    if i == 75:
        print line
        break
fp.close()
"""
