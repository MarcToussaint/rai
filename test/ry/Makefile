run: run/1-basics.ipynb

ipynb_paths =  $(shell find . -type f -name '*.ipynb' -not -name '*checkpoint*' -printf "%f ")

clean: $(ipynb_paths:%=clean/%)

clean/%.ipynb: %.ipynb
	+@-jupyter-nbconvert --ClearOutputPreprocessor.enabled=True --inplace $<
#jupyter-nbconvert --clear-output --inplace $<

run/%.ipynb: %.ipynb
	+@-jupyter-nbconvert --execute $<
