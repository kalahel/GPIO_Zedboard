TARGET=GPIODMC
RECEIPTS=lib$(TARGET)

$(RECEIPTS).a : $(TARGET).o
	ar rc $@ $^
	ranlib $(RECEIPTS).a

$(TARGET).o : $(TARGET).c $(TARGET).h
	gcc -c -std=c99 $<

clean:
	rm -rf *.o

mrproper: clean
	rm -rf $(RECEIPTS).a