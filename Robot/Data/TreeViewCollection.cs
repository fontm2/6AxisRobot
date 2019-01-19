using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Collections.ObjectModel;

namespace Data
{

    public class TreeViewCollection: ObservableCollection<TreeViewNode>
    {
        public TreeViewCollection()
        {

        }
        public void CreateNode(string NodeName)
        {
            this.Add(new TreeViewNode(NodeName));
        }
        public void AddSubNode(string NodeName, string SubNodeName)
        {
            //TreeViewNode MainNode =
            //     (TreeViewNode)(from Node in this
            //                    where Node.NodeName == NodeName
            //                    select Node);
            //MainNode.SubNode.Add(new SubNode(SubNodeName));
            foreach (TreeViewNode Node in this)
            {
                if (Node.NodeName == NodeName)
                {
                    Node.SubNodes.Add(new SubNode(SubNodeName));
                }
            }           
        }       
    }

    public class TreeViewNode
    {
        public string NodeName { get; set; }
        public ObservableCollection<SubNode> SubNodes = new ObservableCollection<SubNode>();
        public TreeViewNode(string nodename)
        {
            this.NodeName = nodename;
        }    
        public ObservableCollection<SubNode> SubNode
        {
            get
            {
                return SubNodes;
            }
            set
            {
                SubNodes = value;
            }
        }
    }
    
    public class SubNode
    {
        private string subnodename;
        public SubNode(string _SubNodeName)
        {
            subnodename = _SubNodeName;
        }
        public string SubNodeName
        {
            get { return subnodename; }
            set { subnodename = value; }
        }
    }
}
