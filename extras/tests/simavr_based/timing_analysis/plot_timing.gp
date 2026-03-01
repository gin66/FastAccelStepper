set terminal pngcairo size 1200,500 enhanced font 'Arial,10'
set output 'timing_comparison.png'

set multiplot layout 1,2

set title "Max High Time: Actual vs Expected" font ',12'
set xlabel "Expected (us)" font ',10'
set ylabel "Actual (us)" font ',10'
set grid
set key bottom right
set size ratio 1

max_val = 45
set xrange [0:max_val]
set yrange [0:max_val]

plot [0:max_val] x with lines lc rgb 'red' lw 1 title 'y=x (limit)', \
     'scatter_max_high.dat' using 1:2 with points pt 7 ps 1.5 lc rgb '#4CAF50' title 'Actual', \
     '' using 1:2:3 with labels offset 1,0.5 font ',7' left notitle

set title "Total High Time: Actual vs Expected" font ',12'
set xlabel "Expected (us)" font ',10'
set ylabel "Actual (us)" font ',10'
set size ratio 1

max_val = 550000
set xrange [0:max_val]
set yrange [0:max_val]

plot [0:max_val] x with lines lc rgb 'red' lw 1 title 'y=x (limit)', \
     'scatter_total.dat' using 1:2 with points pt 7 ps 1.5 lc rgb '#2196F3' title 'Actual', \
     '' using 1:2:3 with labels offset 1,0.5 font ',7' left notitle

unset multiplot
