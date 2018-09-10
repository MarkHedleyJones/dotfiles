set nocompatible              " required
filetype off                  " required

" set the runtime path to include Vundle and initialize
set encoding=utf-8
set rtp+=~/.vim/bundle/Vundle.vim
set rtp+=~/.fzf
set mouse=r
set nu
set clipboard=unnamed
"set tags+=.git/tags
set shell=/bin/bash
set autoindent
set smartindent
set tabstop=4
set shiftwidth=4
"set expandtab
call vundle#begin()

" alternatively, pass a path where Vundle should install plugins
"call vundle#begin('~/some/path/here')

" let Vundle manage Vundle, required
Plugin 'gmarik/Vundle.vim'
Plugin 'vim-scripts/indentpython.vim'
"Plugin 'vim-syntastic/syntastic'
Plugin 'nvie/vim-flake8'
Plugin 'scrooloose/nerdtree'
Plugin 'altercation/vim-colors-solarized'
Plugin 'jnurmine/Zenburn'
Plugin 'kien/ctrlp.vim'
Plugin 'tpope/vim-fugitive'
Plugin 'majutsushi/tagbar' 
Plugin 'Lokaltog/powerline', {'rtp': 'powerline/bindings/vim/'}
Plugin 'MattesGroeger/vim-bookmarks'
Plugin 'w0ng/vim-hybrid'
"Plugin 'kristijanhusak/vim-hybrid-material'


" add all your plugins here (note older versions of Vundle
" used Bundle instead of Plugin)
Bundle 'Valloric/YouCompleteMe'

" ...

" All of your Plugins must be added before the following line
call vundle#end()            " required
filetype plugin indent on    " required

let g:tagbar_autoclose = 1
let g:tagbar_autofocus = 1
let g:tagbar_sort = 0
let g:tagbar_width = 80

let NERDTreeIgnore=['\.pyc$', '\~$'] "ignore files in NERDTree
let python_highlight_all=1
let spelllang="en_gb"

nmap <F7> :TagbarToggle<CR>
nmap <F6> :set spell!<CR>
nmap <F2> :w<CR>
nmap <F10> :set number!<CR>
nmap <F9> <Esc>:w<CR>:!dot -Tpdf wiring.gv -o wiring.pdf<CR>
set t_Co=256
syntax on

highlight BookmarkSign ctermbg=NONE ctermfg=160
highlight BookmarkLine ctermbg=194 ctermfg=NONE
let g:bookmark_sign = '♥'
let g:bookmark_highlight_lines = 1


" if has('gui_running')
" 	set background=dark
" 	colorscheme solarized
" else
" 	colorscheme zenburn
" endif

set background=dark
colorscheme hybrid

"let g:enable_bold_font = 1
"let g:enable_italic_font = 1
"let g:hybrid_transparent_background = 1
"set background=dark
"colorscheme hybrid_material

let g:ycm_confirm_extra_conf=0


"au BufNewFile,BufRead *.cpp, *.hpp
"    \ set tabstop=4       |
"    \ set softtabstop=4   |
"    \ set shiftwidth=4    |
"    \ set textwidth=120   |
"    \ set expandtab       |
"    \ set autoindent      |

au BufNewFile,BufRead *.py
    \ set tabstop=4       |
    \ set softtabstop=4   |
    \ set shiftwidth=4    |
"    \ set textwidth=79    |
    \ set expandtab       |
    \ set autoindent      |
    \ set fileformat=unix |

au BufNewFile,BufRead *.js, *.html, *.css
    \ set tabstop=2       |
    \ set softtabstop=2   |
    \ set shiftwidth=2    |

"au BufRead,BufNewFile *.py,*.pyw,*.c,*.h match BadWhitespace /\s\+$/


let g:ycm_autoclose_preview_window_after_completion=1
map <leader>g  :YcmCompleter GoToDefinitionElseDeclaration<CR>

map <C-n> :NERDTreeToggle<CR>
autocmd bufenter * if (winnr("$") == 1 && exists("b:NERDTree") && b:NERDTree.isTabTree()) | q | endif
