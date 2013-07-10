

class HistoryQueue:
    def __init__(self,maxlength,skip):
        self.content = []
        self.maxlength = maxlength
        self.counter = 0
        self.skip = skip

    def length(self):
        return len(self.content)

    def front(self):
        return self.content[-1]

    def back(self):
        return self.content[0]

    def pop_back(self):
        self.content = self.content[-(len(self.content)-1):]

    def getall(self):
        return self.content

    def push(self,elem):
        self.counter += 1
        if self.counter >= self.skip:
            self.counter = 0
            self.content.append(elem)
            if len(self.content) > self.maxlength:
                self.content = self.content[-self.maxlength:]

    def clear(self):
        self.counter = 0
        self.content = []


class HistoryQueueHD:
    def __init__(self,hdlength,maxlength,skip):
        self.hdlength = hdlength
        self.fastq = HistoryQueue(hdlength,0)
        self.normalq = HistoryQueue(maxlength,skip)

    def push(self,elem):
        if self.fastq.length() >= self.hdlength:
            self.normalq.push(self.fastq.back())
            self.fastq.pop_back()
        self.fastq.push(elem)


    def front(self):
        return self.fastq.front()

    def back(self):
        if self.normalq.length()>0:
            return self.normalq.back()
        else:
            return self.fastq.back()

    def length(self):
        return self.normalq.length() + self.fastq.length()

    def clear(self):
        self.fastq.clear()
        self.normalq.clear()

    def getall(self):
        return self.normalq.getall() + self.fastq.getall()



