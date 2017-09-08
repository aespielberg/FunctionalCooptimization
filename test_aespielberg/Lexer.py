from ply import lex, yacc
class Lexer(object):
  tokens = ('INTEGER', 'IDENTIFIER', 'REAL', 'DIV', 'WORD')
  literals = '+ - * ( ) ^ /'.split()
  t_ignore = ' \t\n'
  t_INTEGER = r'[1-9][0-9]*'
  t_IDENTIFIER = r'[a-z]+[0-9]+'
  t_REAL = r'[0-9]+\.?[0-9]+(e-?[0-9]+)?'
  t_DIV = r'[0-9]+(\.[0-9]+)?/[0-9]+(\.[0-9]+)?'
  t_WORD = r'([A-Z]_?)+'

  def t_error(self, t):
    print("Illegal character '%s'" % t.value[0])
    t.lexer.skip(1)

  def build(self, **kwargs):
    self.lexer = lex.lex(module=self)

# Parser starts here

tokens = Lexer.tokens
start = 'expr'

def p_unit(p):
  '''val  : IDENTIFIER
     val  : REAL
     val  : DIV
     val  : WORD
     term : val      
     prod : term    
     expr : prod
  '''
  p[0] = p[1]

def p_paren(p):
  '''val  : '(' expr ')' '''
  p[0] = '(' + p[2] + ')'

def p_binary(p):
  '''expr : expr '+' prod
     expr : expr '-' prod
     expr : expr '/' prod
     prod : term '*' prod
  '''
  p[0] = '%s%s%s' %(p[1], p[2], p[3])

def p_pow(p):
  '''term : val '^' INTEGER'''
  p[0] = 'pow(%s, %s)' % (p[1], p[3])

def p_unary(p):
  '''term : '-' val'''
  p[0] = '(-%s)' % p[2]

parser = yacc.yacc()
lexer = Lexer().build()
def parse(text):
  return parser.parse(text, lexer=lexer)

if __name__ == '__main__':
  from sys import argv
  for text in argv[1:]:
    print(text + ' => ' + parse(text))
