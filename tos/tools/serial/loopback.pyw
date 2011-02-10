#!/usr/bin/env python

# RS232 loopback
# hacked by Arno Puder, 11/2003

from Tkinter import *
import thread, socket, sys, time, os



class StatusWindow (Frame):
    def __init__ (self, parent=None, text=''):
        Frame.__init__ (self, parent)
        self.pack (expand=YES, fill=BOTH)
        self.makewidgets()

    def makewidgets (self):
        sbar = Scrollbar (self)
        text = Text (self, height=5, width=40, relief=SUNKEN)
        sbar.config (command=text.yview)
        text.config (yscrollcommand=sbar.set)
        sbar.pack (side=RIGHT, fill=Y)
        text.pack (side=LEFT, expand=YES, fill=BOTH)
        self.text = text
        self.clear()

    def clear (self):
        self.empty = 1
        self.text.config (state='normal')
        self.text.delete ('1.0', END)
        self.text.config (state='disabled')

    def log (self, msg):
        self.text.config (state='normal')
        self.text.insert (END, msg)
        self.text.yview (END)
        self.text.config (state='disabled')
        self.empty = 0
        

class LogWindow (Frame):
    def __init__ (self):
        Frame.__init__ (self)

        self.master.title ('COM1')
        # Title
        title = Label (self, text='Serial Port Loopback')
        title.config (pady=10, font=('times', 30, 'bold italic'))
        title.pack (side=TOP)

        # Exit button
        button_frame = Frame (self)
        button_frame.pack (side=RIGHT, padx=10, pady=10)

        exit = Button (button_frame, text='Exit', command=self.quit)
        exit.pack (side=BOTTOM, fill=BOTH, padx=10, pady=5)

        # Status window
        self.sw = StatusWindow (self)
        self.sw.pack (side=BOTTOM, padx=10, pady=10)

        # Clear button
        clear = Button (button_frame, text='Clear', command=self.sw.clear)
        clear.pack (side=TOP, fill=BOTH, padx=10, pady=5)

        self.pack()
        
    def log (self, msg):
        self.sw.log (msg)
        

def read_data_from_socket (log_window, inet):
    ip_port = inet.split (':')
    if len (ip_port) != 2:
        print 'Bad inet argument (%s)' % inet
        thread.exit_thread()
    ip = ip_port[0]
    port = int (ip_port[1])
    while 42:
        try:
            s = socket.socket (socket.AF_INET, socket.SOCK_STREAM)
            s.bind ((ip, port))
            s.listen (1)
            conn, addr = s.accept()
            # we need to skip the first few bytes and don't
            # display them in the log window. This is because
            # TOS' boot loader automatically sends a command sequence
            # that we simply ignore
            bytes_read = 0
            BYTES_TO_SKIP = 22
            while 42:
                try:
                    ch = conn.recv (1)
                except:
                    conn.close()
                    break
                if not ch:
                    conn.close()
                    break
                try:
                    if ch != '':
                        bytes_read += 1
                        if bytes_read > BYTES_TO_SKIP:
                            log_window.log (ch)
                            conn.send (ch)
                except:
                    break
        except:
            print "Could not connect to '%s'" % inet
            thread.exit_thread()

log_window = LogWindow()

try:
    bochsrc = os.path.expanduser ('~/.bochsrc')
    bochsrc_f = open (bochsrc, 'r')
    lines = bochsrc_f.readlines()
    bochsrc_f.close()
    inet = None
    for l in lines:
        if l[0] == '#':
            continue
        i = l.find ('com1: ')
        if i != -1:
            if l.find('mode=socket') == -1:
                print 'com1 not redirected to a socket'
                continue
            i = l.find ('dev=')
            if i == -1:
                print 'com1 not followed by dev argument'
                continue
            inet = l[i+4:-1]
    if inet == None:
        print 'com1 not configured properly'
        throw
except:
    inet = "localhost:8888"

thread.start_new (read_data_from_socket, (log_window, inet))
mainloop()
