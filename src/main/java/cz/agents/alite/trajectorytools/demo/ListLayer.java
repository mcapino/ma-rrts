package cz.agents.alite.trajectorytools.demo;

import java.awt.Graphics2D;
import java.awt.event.MouseEvent;

import javax.swing.JList;
import javax.swing.ListSelectionModel;
import javax.swing.event.MouseInputAdapter;

import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.vis.Vis;
import cz.agents.alite.vis.layer.AbstractLayer;
import cz.agents.alite.vis.layer.GroupLayer;
import cz.agents.alite.vis.layer.VisLayer;

public class ListLayer extends AbstractLayer {

    public interface SelectionListener {
        void selectedIndex(int index);
    }
    
    public static <V extends SpatialPoint,E> VisLayer create(Object[] data, final SelectionListener listener, final int x, final int y, int width, int height) {
        GroupLayer group = GroupLayer.create();

        final JList list = new JList(data);
        list.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
        list.setBounds(x, y, width, height);
        list.setSelectedIndex(0);
        list.setVisible(true);

        group.addSubLayer( new AbstractLayer() {
            MouseInputAdapter mouseListener;
            @Override
            public void init(Vis vis) {
                mouseListener = new MouseInputAdapter() {

                    @Override
                    public void mouseClicked(MouseEvent e) {
                        super.mouseClicked(e);

                        if (list.contains(e.getX() - x, e.getY() - y)) {
                            int index = list.locationToIndex(new java.awt.Point(e.getX() - x, e.getY() - y));
                            System.out.println("list.locationToIndex(new java.awt.Point(e.getX() - x, e.getY() - y)): " + index);
                            list.setSelectedIndex(index);
                            listener.selectedIndex(index);
                        }
                    }
                };
                vis.addMouseListener(mouseListener);
                super.init(vis);
            }

            @Override
            public void deinit(Vis vis) {
                super.deinit(vis);

                vis.removeMouseListener(mouseListener);
            }

            @Override
            public void paint(Graphics2D canvas) {
                canvas.translate(x, y);
                list.paint(canvas);
                canvas.translate(-x, -y);
            }

            @Override
            public String getLayerDescription() {
                return "List";
            }
        });
        return group;
    }
}