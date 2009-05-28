#!/usr/bin/perl -s

$IOMEM_FILE = "/proc/iomem";
open(IOMEM_FILE) or die("Could not open iomem file.");
$largest_chunk_size = 0;
$largest_chunk_address = 0;
$last_end_address = 0;
foreach $line (<IOMEM_FILE>) {
	# loop over each line, checking to see when we have the largest chunk of memory available
	$first_char = substr( $line, 0, 1);
	next if ($first_char eq ' ');
	chomp($line);
	($addresses, $name) = split(' : ', $line);
	($first_address, $second_address) = split('-', $addresses);
	$first_address = hex($first_address);
	$second_address = hex($second_address);
	$last_size = $first_address-$last_end_address;
	if ($last_size > $largest_chunk_size) {
		$largest_chunk_size = $last_size;
		$largest_chunk_address = $last_end_address;
	}
	$last_end_address = $second_address;
}

# now deal with the fact that it should be 256-MB aligned
if ($largest_chunk_address & 0x0FFFFFFF) {
  $largest_chunk_address -= $largest_chunk_address & 0x0FFFFFFF;
  $largest_chunk_address += 0x10000000;
}
$largest_chunk_size -= $largest_chunk_size & 0x0FFFFFFF;

if ($size) {
  print sprintf("0x%x\n", $largest_chunk_size);
}
if ($address) {
  print sprintf("0x%x\n", $largest_chunk_address);
}
