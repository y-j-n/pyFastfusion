version=$(shell cat ../VERSION)
all: index.html

index.html: index.org style.css 
#http://lists.gnu.org/archive/html/emacs-orgmode/2011-04/msg00220.html
	emacs -nw --batch --visit=$< --funcall=org-html-export-to-html
	perl -pi -e 's/__VERSION__/$(version)/g' index.html
	rm *~

clean:
	rm -f index.html *~
