boost_dep = dependency('boost', modules : ['iostreams', 'iterator', 'geometry', 'spirit'], version : '>=1.58')

src = files('main.cpp')

add_global_arguments('-std=c++11', language : 'cpp')
executable('main', src, dependencies : boost_dep)