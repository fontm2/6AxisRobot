using System;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.IO.Ports;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Threading;
using System.Xml.Serialization;
using System.IO;
using System.Collections.ObjectModel;
using System.Collections;
using System.Diagnostics;
using Data;

namespace FileHandler
{
    public static class CustomFile
    {
        public static ObservableCollection<NCData> ReadXMLListGCode(string filename)
        {
            XmlSerializer reader = new XmlSerializer(typeof(ObservableCollection<NCData>));
            StreamReader file = new StreamReader(filename);
            ObservableCollection<NCData> gcodedata = (ObservableCollection<NCData>)reader.Deserialize(file);          
            file.Close();
            return gcodedata;
        }
        public static void WriteXMLListGCode(ObservableCollection<NCData> gcodedata, string filename)
        {

            XmlSerializer writer = new XmlSerializer(typeof(ObservableCollection<NCData>));
            StreamWriter file = new StreamWriter(filename);
            writer.Serialize(file, gcodedata);
            file.Close();
        }
        public static List<string> ReadText(string filename)
        {
            List<string> list = new List<string>();
            string line;
            StreamReader file = new StreamReader(filename);
            while ((line = file.ReadLine()) != null)
            {
                list.Add(line); // Add to list.                       
            }
            file.Close();
            return list;
        }
    }
}
