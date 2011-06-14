
" Split always below and right
set splitbelow
set splitright

" allow for (1) navigation through buffs without be asked to save and (2)
" remember undo history for not active buffs
set hidden

" Dont fold lines automatically
set textwidth=0

" Highlight search
set hls

" folding according to file type
set foldmethod=syntax

" tabstop and shiftwidth
set expandtab
set ts=2
set sw=2
set sts=2

" make use alternatively on the cluster qmake, in irb  gmake
set makeprg=make

" write automatically when compiling
set autowrite

" omni completion
set omnifunc=ccomplete#Complete

" run
map <F8>  :!make<CR>\ &&\ %:p:h/x.exe<CR>

" get tags file from share and local tags file
set tags+=../tags,../../tags
set tags+=tags


" Here the start layout of the windows
edit main.cpp
