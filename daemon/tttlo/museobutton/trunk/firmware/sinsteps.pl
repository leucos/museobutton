#!/usr/bin/perl

my $PI=3.14159265;

my $steps = shift;
my $range = shift;
my $numbers = 0;

print "uint8_t sine_values[STEPS] EEMEM = { ";

for ($i=0; $i < $steps; $i++) {

  if (($numbers % 8 == 0) && ($numbers != 0)) {
      print ("\n", ' 'x37);
  }

  print int( .5 + ((sin($PI*$i*2/($steps-1)-$PI/2)+1)*($range/2)) );
  print ", ";
  $numbers++;
}


print "};\n";

