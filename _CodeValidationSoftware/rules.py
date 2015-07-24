import sys
import re

def tab_to_space (line, tab_lenght = 4):
	while '\t' in line:
		first_tab_init_pos = line.find('\t')
		first_tab_end_pos = (((first_tab_init_pos // tab_lenght)+1) * tab_lenght)
		diff = first_tab_end_pos - first_tab_init_pos
		if diff == 0:
			spaces_string = ' ' * tab_lenght
		else:
			spaces_string = ' ' * diff
		line = line.replace('\t', spaces_string, 1)
	return line

def comment_remover(text):
	def replacer(match):
		s = match.group(0)
		if s.startswith('/'):
			return " " # note: a space and not an empty string
		else:
			return s
	pattern = re.compile(
		r'//.*?$|/\*.*?\*/|\'(?:\\.|[^\\\'])*\'|"(?:\\.|[^\\"])*"',
		re.DOTALL | re.MULTILINE
	)
	return re.sub(pattern, replacer, text)
	
def checkComments(line, commentedLine):
	commentStart = line.rfind("/*");
	commentEnd = line.rfind("*/");
	if commentStart>commentEnd:
		commentedLine=1;
	if commentStart<commentEnd: 
		commentedLine=0;
	return commentedLine, commentStart, commentEnd;
	
#find } else { or rule to it
	
def check_SpaceAfterKeyword(line, commentStartIdx, commentEndIdx):
	startIndex=-1;
	endIndex=-1;
	pattern = re.compile("[ ]+(if|else)(?! )")
	for m in pattern.finditer(line):
		index=m.start()+len(m.group())-len(m.group().lstrip(' '));
		pattern=m.group().lstrip(' '); 
		startIndex=index;
		endIndex=index+len(pattern);
		if commentStartIdx<=startIndex:
			startIndex=-1;
			endIndex=-1;
		if(endIndex==len(line)-1):
			startIndex=-1;
			endIndex=-1;
		line = add_ErrorIndicator(line, startIndex, endIndex);
	return line;
	
def check_SpaceAfterCommaSemicolon(line, commentStartIdx, commentEndIdx):
	startIndex=-1;
	endIndex=-1;
	offsetFromPreviousErrors=0;
	pattern = re.compile("(,|;)(?! )")
	for m in pattern.finditer(line):
		index=m.start()+len(m.group())-len(m.group().lstrip(' '));
		pattern=m.group().lstrip(' '); 
		startIndex=index;
		endIndex=index+len(pattern);
		if commentStartIdx<=startIndex:
			startIndex=-1;
			endIndex=-1;
		if(endIndex==len(line)-1):
			startIndex=-1;
			endIndex=-1;
		line = add_ErrorIndicator(line, startIndex+offsetFromPreviousErrors, endIndex+offsetFromPreviousErrors);
		offsetFromPreviousErrors+=7;
	return line;
	
def check_NoSpaceBeforeUnaryOperators(line, commentStartIdx, commentEndIdx):
	startIndex=-1;
	endIndex=-1;
	offsetFromPreviousErrors=0;
	pattern = re.compile("(((\+\+|\-\-)( )([a-zA-Z_\(][a-zA-Z0-9_\)\(]*)))|(([a-zA-Z_\(][a-zA-Z0-9_\)]*)( )((\+\+|\-\-)))|((\!|\~)( ))")
	for m in pattern.finditer(line):
		index=m.start()+len(m.group())-len(m.group().lstrip(' '));
		pattern=m.group().lstrip(' '); 
		startIndex=index;
		endIndex=index+len(pattern);
		if commentStartIdx<=startIndex:
			startIndex=-1;
			endIndex=-1;
		if(endIndex==len(line)-1):
			startIndex=-1;
			endIndex=-1;
		line = add_ErrorIndicator(line, startIndex+offsetFromPreviousErrors, endIndex+offsetFromPreviousErrors);
		offsetFromPreviousErrors+=7;
	return line;
	
def check_SpaceBeforeAndAfterNonUnaryOperators(line, commentStartIdx, commentEndIdx):
	startIndex=-1;
	endIndex=-1;
	offsetFromPreviousErrors=0;
	pattern = re.compile("((?! )|(\+))(\+|\-|=)((?![ |=])|(\+))")
	for m in pattern.finditer(line):
		index=m.start()+len(m.group())-len(m.group().lstrip(' '));
		pattern=m.group().lstrip(' '); 
		startIndex=index;
		endIndex=index+len(pattern);
		if commentStartIdx<=startIndex:
			startIndex=-1;
			endIndex=-1;
		line = add_ErrorIndicator(line, startIndex+offsetFromPreviousErrors, endIndex+offsetFromPreviousErrors);
		offsetFromPreviousErrors+=7;
	return line;

def add_ErrorIndicator(line, startIndex, endIndex):
	if startIndex != -1:
		line = line[:endIndex] + '</a>' + line[endIndex:];
		line = line[:startIndex] + '<a>' + line[startIndex:];
	return line;
	
def checkLine(line, commentedLine):
	commentedLine, startIdx, endIdx = checkComments(line, commentedLine);
	if commentedLine==0:
		if startIdx==-1 and endIdx==-1:
			startIdx=100;
		line = check_SpaceAfterKeyword(line, startIdx, endIdx); 
		line = check_SpaceAfterCommaSemicolon(line, startIdx, endIdx);
		line = check_NoSpaceBeforeUnaryOperators(line, startIdx, endIdx);
		line = check_SpaceBeforeAndAfterNonUnaryOperators(line, startIdx, endIdx);
	return line, commentedLine
	
def main(argv):
	testFile = open('test.c','r');
   
	commentedLine=0;
   
	resultFile = open('result.html','w');
	resultFile.write('<html><body>\n');
	resultFile.write('<head><link rel="stylesheet" type="text/css" href="style.css"></head>\n');
	for line in testFile:
		line = tab_to_space(line);
		#line = comment_remover(line);
		line, commentedLine = checkLine(line, commentedLine);
		resultFile.write('<pre>'+line+'</pre>');
	resultFile.write('\n</body></html>');
	pass

if __name__ == "__main__":
	main(sys.argv)