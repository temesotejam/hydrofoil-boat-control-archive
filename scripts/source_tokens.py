"""C++ lexical comparison: ignore formatting, retain literals and token boundaries."""
import re

LEXER = re.compile(
    r'(?P<space>\s+)|(?P<comment>//[^\n]*|/\*[\s\S]*?\*/)' 
    r'|(?P<raw>(?:u8|u|U|L)?R"(?P<delimiter>[^ ()\\\t\r\n]{0,16})\([\s\S]*?\)(?P=delimiter)")'
    r'|(?P<string>(?:u8|u|U|L)?"(?:\\[\s\S]|[^"\\])*")'
    r"|(?P<char>(?:u8|u|U|L)?'(?:\\[\s\S]|[^'\\])*')"
    r'|(?P<identifier>[A-Za-z_$][A-Za-z_0-9$]*)'
    r"|(?P<number>(?:\d|\.\d)(?:[A-Za-z_0-9.']|[eEpP][+-])*)"
    r'|(?P<operator>%:%:|>>=|<<=|<=>|->\*|\.\.\.|::|->|\.\*|\+\+|--|<<|>>|<=|>=|==|!=|&&|\|\||\*=|/=|%=|\+=|-=|&=|\^=|\|=|##|<:|:>|<%|%>|%:|.)'
)

def tokens(text):
    # Translation phase 2 removes backslash-newline before recognizing tokens.
    text = text.replace('\\\r\n', '').replace('\\\n', '')
    result = []
    pos = 0
    for m in LEXER.finditer(text):
        if m.start() != pos:
            raise ValueError(f'unrecognized source at {pos}')
        pos = m.end()
        if m.lastgroup not in ('space', 'comment'):
            result.append(m.group())
    if pos != len(text):
        raise ValueError(f'unrecognized source at {pos}')
    return result
