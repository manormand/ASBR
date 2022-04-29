clear; clc
link = input('link? \n -> -> ','s');

idx = regexp(link, '/[0-9]') + 1;
doc_num = link(idx:(idx+6));


path = 'https://ieeexplore-ieee-org.ezproxy.lib.utexas.edu/document/';

link = [path doc_num];
fprintf('New Link:\n -> -> %s\n', link)
clipboard('copy',link)