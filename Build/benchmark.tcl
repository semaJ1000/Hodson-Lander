#source benchmark.tcl

# Test script for performance measurements

# Test with different particle counts
puts "\nTesting with 10 particles"
system lander dim 10
simulator landerSim benchmark 1000

puts "\nTesting with 50 particles"
system lander dim 50
simulator landerSim benchmark 1000

puts "\nTesting with 100 particles"
system lander dim 100
simulator landerSim benchmark 1000

puts "\nTesting with 1000 particles"
system lander dim 1000
simulator landerSim benchmark 1000

# Test with different box configurations
puts "\nTesting with sparse box layout"
system lander sparseBlockLayout
simulator landerSim benchmark 1000

puts "\nTesting with many block layout"
system lander manyBlockLayout
simulator landerSim benchmark 1000