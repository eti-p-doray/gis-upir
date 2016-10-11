boost_dep = dependency('boost', modules : ['iostreams', 'iterator', 'geometry', 'spirit'], version : '>=1.58')

add_global_arguments('-std=c++11', language : 'cpp')

src = files('main.cpp')

executable('main', src, dependencies : boost_dep)

test('csv_unittest', executable('csv_unittest', 'csv_unittest.cpp', dependencies : boost_dep))